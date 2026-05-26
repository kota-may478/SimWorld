"""Simulate Cursor/Jupyter kernel startup via kernelspec."""
from jupyter_client.manager import KernelManager

km = KernelManager(kernel_name="simworld")
print("starting kernel...")
km.start_kernel()
print("kernel started:", km.kernel_spec.argv)

kc = km.client()
kc.wait_for_ready(timeout=30)
print("kernel ready")

msg_id = kc.execute("import sys; print('connect ok', sys.executable)")
while True:
    msg = kc.get_iopub_msg(timeout=30)
    if msg["parent_header"].get("msg_id") != msg_id:
        continue
    msg_type = msg["header"]["msg_type"]
    if msg_type == "stream":
        print("output:", msg["content"]["text"], end="")
    elif msg_type == "status" and msg["content"]["execution_state"] == "idle":
        break
    elif msg_type == "error":
        print("error:", msg["content"])
        break

km.shutdown_kernel(now=True)
print("done")
