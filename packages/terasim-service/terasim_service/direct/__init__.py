"""Direct (Redis/HTTP-free) co-simulation transport.

gRPC contract between TeraSimCoSimDirectPlugin (server, inside the TeraSim
process) and CarlaCosim (client). See cosim_direct.proto for the contract and
plugins/cosim_direct.py for the server implementation.
"""
