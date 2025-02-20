import pyudev
from typing import Optional

def find_device_port(vendor_id: str, product_id: str, default: Optional[str] = None) -> Optional[str]:
    context = pyudev.Context()
    
    for device in context.list_devices(subsystem='tty'):
        if device.get('ID_VENDOR_ID') == vendor_id and device.get('ID_MODEL_ID') == product_id:
            return device.device_node
    
    return default
