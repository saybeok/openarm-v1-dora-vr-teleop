import dora
import numpy as np
import pyarrow as pa


def main():
    node = dora.Node()

    right = None
    left = None

    node.send_output("status", pa.array(["ready"]))

    for event in node:
        if event["type"] != "INPUT":
            continue

        eid = event["id"]

        if eid == "right":
            values = event["value"].to_numpy().astype(np.float32)
            if values.shape == (8,):
                right = values

        elif eid == "left":
            values = event["value"].to_numpy().astype(np.float32)
            if values.shape == (8,):
                left = values

        elif eid == "tick":
            pass

        if right is not None and left is not None:
            pos16 = np.concatenate([right, left]).astype(np.float32)
            node.send_output("position", pa.array(pos16, type=pa.float32()))


if __name__ == "__main__":
    main()
