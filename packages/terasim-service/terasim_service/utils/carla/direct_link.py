"""Client side of the direct (Redis/FastAPI-free) co-simulation link.

Wraps the CosimDirect gRPC stub for CarlaCosim. One tick_async() call carries
this step's agent commands, asks the TeraSim-side plugin to run exactly one
SUMO step, and resolves to the post-step state — replacing the Redis-era
sequence of agent_command POST + tick POST + status polling + states GET.
"""

import json
import time

import grpc

from ...direct import cosim_direct_pb2, cosim_direct_pb2_grpc

_MSG_OPTIONS = [
    ("grpc.max_send_message_length", 128 * 1024 * 1024),
    ("grpc.max_receive_message_length", 128 * 1024 * 1024),
]


class DirectLink:
    """gRPC connection to a TeraSimCoSimDirectPlugin server."""

    def __init__(self, address: str, ready_timeout: float = 600.0):
        """Connect and wait until the simulation reaches wait_for_tick.

        ready_timeout is generous because the TeraSim runner may still be
        loading the SUMO network / inserting warmup traffic when the client
        starts.
        """
        self.address = address
        self.channel = grpc.insecure_channel(address, options=_MSG_OPTIONS)
        self.stub = cosim_direct_pb2_grpc.CosimDirectStub(self.channel)

        deadline = time.time() + ready_timeout
        last_status = "unreachable"
        while True:
            try:
                resp = self.stub.GetState(
                    cosim_direct_pb2.GetStateRequest(), timeout=2.0
                )
                last_status = resp.status
                if resp.status in ("wait_for_tick", "ticked", "running"):
                    break
                if resp.status in ("finished", "error"):
                    raise RuntimeError(
                        f"TeraSim at {address} already ended (status={resp.status})"
                    )
            except grpc.RpcError:
                pass  # server not up yet
            if time.time() > deadline:
                raise TimeoutError(
                    f"TeraSim direct link at {address} not ready within "
                    f"{ready_timeout:.0f}s (last status: {last_status})"
                )
            time.sleep(0.5)
        print(f"TeraSim direct link ready at {address} (status={last_status})")

    def tick_async(self, commands):
        """Request one SUMO step (non-blocking).

        commands: list of dicts {agent_id, agent_type, command_type, data}.
        Returns a grpc Future resolving to a TickResponse.
        """
        request = cosim_direct_pb2.TickRequest(
            commands=[
                cosim_direct_pb2.AgentCommand(
                    agent_id=c["agent_id"],
                    agent_type=c["agent_type"],
                    command_type=c["command_type"],
                    data_json=json.dumps(c.get("data", {})),
                )
                for c in commands
            ]
        )
        return self.stub.Tick.future(request, timeout=300.0)

    def get_state(self):
        """Fetch the latest state without advancing the simulation."""
        return self.stub.GetState(cosim_direct_pb2.GetStateRequest(), timeout=10.0)

    def stop(self):
        """Ask the simulation to stop; ignore errors (it may already be gone)."""
        try:
            self.stub.Stop(cosim_direct_pb2.StopRequest(), timeout=5.0)
        except grpc.RpcError:
            pass

    def close(self):
        self.channel.close()


def parse_state_json(state_json: str):
    """TickResponse.state_json -> dict (same schema as the HTTP states payload)."""
    if not state_json:
        return None
    return json.loads(state_json)
