import attach_needle
from ambf_client import Client
import psm_arm

'''
copy from attach_needle
'''
c = Client('attach_needle')
c.connect()
# psm_name =
needle = c.get_obj_handle('Needle')
link1 = c.get_obj_handle('psm1' + '/toolyawlink')
link2 = c.get_obj_handle('psm2' + '/toolyawlink')
link3 = c.get_obj_handle('psm3' + '/toolyawlink')
attach_needle.attach_needle(needle, link1)

test = PSM(c, 'test')
test.run_grasp_logic(0.15)
