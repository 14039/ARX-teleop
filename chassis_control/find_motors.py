import socket
import canopen
import os
import time

# CAN bus configuration (may be overridden via environment variables)
CAN_CHANNEL: str = os.getenv("CAN_CHANNEL", "can1")
CAN_BITRATE: int = int(os.getenv("CAN_BITRATE", "1000000"))  # 1 Mbps


def main() -> None:
    """Scans the CAN bus for available nodes and prints their IDs."""
    network = canopen.Network()  # type: ignore[attr-defined]
    print(f"Connecting to CAN bus ({CAN_CHANNEL}, {CAN_BITRATE} bit/s)…")
    try:
        network.connect(
            bustype="socketcan",
            channel=CAN_CHANNEL,
            bitrate=CAN_BITRATE,
        )

        print("Scanning for nodes…")
        network.scanner.search(limit=127)

        # Give nodes some time to respond to the search request
        time.sleep(2)

        if not network.scanner.nodes:
            print("No nodes found on the network.")
        else:
            print("Found the following node IDs:")
            for node_id in sorted(network.scanner.nodes):
                print(f"- {node_id}")

    except Exception as e:
        print(f"An error occurred: {e}")
        print(
            "Please ensure the CAN interface is configured correctly "
            f"(e.g., `sudo ip link set {CAN_CHANNEL} up type can bitrate {CAN_BITRATE}`)"
        )
    finally:
        network.disconnect()
        print("Done.")


if __name__ == "__main__":
    main()
