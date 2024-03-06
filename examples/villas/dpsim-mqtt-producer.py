import paho.mqtt.publish as publish
import json
import time


def build_message(sequence, v_ref):
    ts_field = {"origin": [int(elem) for elem in str(time.time()).split(".")]}
    data_field = [v_ref]
    message = {"ts": ts_field, "sequence": sequence, "data": data_field}

    return "[" + json.dumps(message) + "]"


if __name__ == "__main__":
    time.sleep(10)
    seq = 0
    T_s = 0.01
    tf = 10.0
    num_samples = int(tf / T_s) + 2
    for n in range(0, num_samples):
        if n < int(num_samples / 2):
            m_v_ref = {"real": 5.0, "imag": 0.0}
        else:
            m_v_ref = {"real": 7.0, "imag": 0.0}
        m_message = build_message(sequence=seq, v_ref=m_v_ref)
        print(m_message)
        publish.single(topic="/mqtt-dpsim", payload=m_message, hostname="mqtt")
        seq += 1
        time.sleep(T_s)
