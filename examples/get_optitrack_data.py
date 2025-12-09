import zenoh, time
import msgpack

def listener(sample):
    print(f"{sample.key_expr}{msgpack.unpackb(sample.payload.to_bytes(), raw=False)}")

if __name__ == "__main__":
    with zenoh.open(zenoh.Config()) as session:
        sub = session.declare_subscriber('*/pose', listener)
        time.sleep(60)