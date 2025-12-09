import zenoh, time
import msgpack

if __name__ == "__main__":
    with zenoh.open(zenoh.Config()) as session:
        key = 'turtle5/cmd_vel'
        pub = session.declare_publisher(key)
        while True:
            msg = {
                "linear": {
                    "x": 0.2,
                    "y": 0.0,
                    "z": 0.0
                },
                "angular": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": -0.5
                }
            }
            msg_packed = msgpack.packb(msg, use_bin_type=True)
            print(msg_packed)
            pub.put(msg_packed)
            time.sleep(0.1)