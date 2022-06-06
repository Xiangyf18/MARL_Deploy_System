import netifaces
from numpy import result_type
def get_mac_address():
    try:
        result=netifaces.ifaddresses('wlan0')[netifaces.AF_LINK]
    except:
        result=netifaces.ifaddresses('enp0s31f6')[netifaces.AF_LINK]# on Master Computer
    return result[0]['addr']

'''
import uuid
def get_mac_address(): 
    mac=uuid.UUID(int = uuid.getnode()).hex[-12:] 
    mac_2=uuid.uuid4().hex[-12:]        
    return ":".join([mac[e:e+2] for e in range(0,11,2)])
'''

def mac2id(mac):
    mac2id_table = {}
    mac2id_table['e4:5f:01:00:d4:20'] = 'AKM_1'
    mac2id_table['e4:5f:01:00:d3:d8'] = 'AKM_2'
    mac2id_table['e4:5f:01:00:d4:5c'] = 'AKM_3'
    mac2id_table['e4:5f:01:1b:a2:39'] = 'AKM_4'
    mac2id_table['e4:5f:01:1b:9e:ca'] = 'AKM_5'

    mac2id_table['68:ec:c5:4d:71:e6'] = 'MKN_1'
    mac2id_table['f8:59:71:52:69:2b'] = 'MKN_2'
    #mac2id_table['06:28:77:d1:50:a1'] = 'MKN_2'
    mac2id_table['f8:59:71:98:98:47'] = 'MKN_3'
    mac2id_table['84:5c:f3:27:50:f6'] = 'MKN_4'
    mac2id_table['84:5c:f3:27:50:d8'] = 'MKN_5'
    
    mac2id_table['e0:d5:5e:26:58:85'] = 'host_0'#
    
    if mac in mac2id_table.keys():
        return mac2id_table[mac]
    else:
        return 'dummy'

if __name__ == '__main__':
    print(mac2id(get_mac_address()))
    