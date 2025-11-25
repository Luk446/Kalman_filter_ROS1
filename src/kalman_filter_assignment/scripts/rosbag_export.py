import csv
import os
import rosbag
from typing import Any, Dict

bagfile = "kf_test.bag"

topics = [
    '/kalman_estimate',
    '/odom',
    '/fake_gps',
    '/odom1',
    '/cmd_vel'
]

def ros_msg_to_dict(msg: Any, prefix: str = "") -> Dict[str, Any]:
    """Recursively flattens a ROS message into a dict of primitive fields.
    Arrays of primitives kept as list; arrays of messages expanded with index suffix.
    """
    flat: Dict[str, Any] = {}
    # Odometry etc. have __slots__
    if hasattr(msg, '__slots__'):
        for slot in msg.__slots__:
            val = getattr(msg, slot)
            key_base = f"{prefix}{slot}".rstrip('_')
            # Primitive types
            if isinstance(val, (int, float, str, bool)) or val is None:
                flat[key_base] = val
            # List or tuple
            elif isinstance(val, (list, tuple)):
                if len(val) == 0:
                    flat[key_base] = []
                else:
                    # If first element is a ROS message, recurse with indices
                    if hasattr(val[0], '__slots__'):
                        for i, elem in enumerate(val):
                            flat.update(ros_msg_to_dict(elem, f"{key_base}{i}_"))
                    else:
                        flat[key_base] = list(val)
            # Nested ROS message
            elif hasattr(val, '__slots__'):
                flat.update(ros_msg_to_dict(val, f"{key_base}_"))
            else:
                # Fallback: store repr
                flat[key_base] = repr(val)
    else:
        flat[prefix.rstrip('_') or 'value'] = msg
    return flat


def export_topic(topic: str) -> int:
    """Export a single topic to CSV. Returns number of rows written."""
    csvname = topic.strip('/').replace('/', '_') + ".csv"
    rows_written = 0
    csvfile = None
    writer = None

    try:
        with rosbag.Bag(bagfile, 'r') as bag:
            for _, msg, t in bag.read_messages(topics=[topic]):
                flat = ros_msg_to_dict(msg)

                if writer is None:
                    csvfile = open(csvname, 'w', newline='')
                    header = ['time'] + sorted(flat.keys())
                    writer = csv.DictWriter(
                        csvfile,
                        fieldnames=header,
                        extrasaction='ignore'
                    )
                    writer.writeheader()

                row = {'time': t.to_sec()}
                row.update(flat)
                writer.writerow(row)
                rows_written += 1
    finally:
        if csvfile is not None:
            csvfile.close()

    if rows_written == 0:
        if os.path.exists(csvname):
            os.remove(csvname)
        print(f"[WARN] Topic {topic} had no messages in {bagfile}; CSV skipped.")
    else:
        print(f"[INFO] Wrote {rows_written} rows to {csvname} from {topic}.")

    return rows_written


def main() -> None:
    for topic in topics:
        export_topic(topic)


if __name__ == "__main__":
    main()