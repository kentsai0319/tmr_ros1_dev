#!/usr/bin/env python

import rospy
from tmr_msgs.srv import *

import os
import shutil
import sys

from modify_urdf import *

def _gen_urdf():
    rospy.init_node('modify_urdf')

    if len(sys.argv) < 3:
        rospy.logwarn('usage: modify_urdf_node model new_model replace')
        return

    model = sys.argv[1]
    new_model = sys.argv[2]
    replace = False
    if len(sys.argv) == 4:
        if sys.argv[3].upper() == 'REPLACE':
            replace = True
            rospy.logwarn('origin urdf file will be replaced')

    rospy.wait_for_service('tmr/ask_item')
    ask_item = rospy.ServiceProxy('tmr/ask_item', AskItem)
    res_dh = ask_item('dh', 'DHTable', 1.0)
    res_dd = ask_item('dd', 'DeltaDH', 1.0)

    if not res_dh.value.startswith('DHTable={') or not res_dh.value.endswith('}'):
        rospy.logerr('invalid dh')
        return
    if not res_dd.value.startswith('DeltaDH={') or not res_dd.value.endswith('}'):
        rospy.logerr('invalid delta_dh')
        return

    rospy.loginfo(res_dh.value)
    rospy.loginfo(res_dd.value)

    dh_strs = res_dh.value[9:-1].split(',')
    dd_strs = res_dd.value[9:-1].split(',')

    if len(dh_strs) != 42:
        rospy.logerr('invalid dh')
        return
    if len(dd_strs) != 30:
        rospy.logerr('invalid delta_dh')
        return

    dh = [float(i) for i in dh_strs]
    dd = [float(i) for i in dd_strs]

    urdf_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/urdf'

    urdf_name = '/' + model + '.urdf'
    new_urdf_name = '/' + new_model + '.urdf'

    file_in = urdf_path + urdf_name
    file_out = urdf_path + new_urdf_name

    rospy.loginfo(file_in)

    fr = open(file_in, 'r')
    link_data = fr.read()
    fr.close()

    root = ET.fromstring(link_data)

    udh = urdf_DH_from_tm_DH(dh, dd)
    xyzs, rpys = xyzrpys_from_urdf_DH(udh)
    modify_urdf(root, xyzs, rpys, udh)

    link_data = ET.tostring(root, encoding='UTF-8')

    file_save = ''
    if replace:
        file_save = file_in
        rospy.loginfo('copy and rename origin urdf file')
        shutil.copyfile(file_in, file_out)
    else:
        file_save = file_out

    rospy.loginfo(file_save)

    fw = open(file_save, 'w')
    fw.write(link_data)
    fw.close()

def main():
    try:
        _gen_urdf()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
