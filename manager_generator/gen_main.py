#!/usr/bin/python3

import argparse
import os
import sys

parser = argparse.ArgumentParser(description="generate a resource manager node")
parser.add_argument("--package-name",metavar="PKG", type=str, required=True,
        help="name of the project/package")
parser.add_argument("--target-types", type=str, metavar="name:data_type", nargs='+', help="", required=True)
parser.add_argument("--priority-topics", type=str, metavar="topic_name:msg_type", nargs='+', help="msg_type is one defined in --priority_msgs")

args=parser.parse_args()
generator_dir=os.path.dirname(sys.argv[0])
in_msg_dir = os.path.join(generator_dir,"cmake","gen","msg")

print(args)

#package architecture
os.makedirs(os.path.join(args.package_name,"src"))
os.makedirs(os.path.join(args.package_name,"msg"))


# CmakeLists.txt
# TODO
# package.xml
# TODO

# .msg files

#   CoordinationState
for x in args.target_types :
    name=x.split(':')[0]
    data_type=x.split(':')[1]
    f = open(os.path.join(args.package_name,'msg','CoordinationState'+name+'.msg'),'w')
    f.write("CoordinationStateHeader header\n")
    f.write("{} data\n".format(data_type))
    f.close()

#   PriorityTarget
for x in args.target_types :
    name=x.split(':')[0]
    data_type=x.split(':')[1]
    f = open(os.path.join(args.package_name,'msg','Priority'+name+'.msg'),'w')
    f.write("PriorityTargetHeader priority\n")
    f.write("{} data\n".format(data_type))
    f.close()


#   CoordinationSignal
f_signal=open(os.path.join(args.package_name,'msg','CoordinationSignal.msg'),'w')
f_signal.write("EndCondition end_condition # coordination signal must stop whatever state it is in if this condition is verified\n")
for x in args.target_types :
    name=x.split(':')[0]
    data_type=x.split(':')[1]
    f_signal.write("{} states_{}[]\n".format('CoordinationState'+name,data_type))


# main node code:

mainfile=os.path.join(args.package_name, "src", args.package_name+".cpp")

fo = open(mainfile,"w")

# include message headers
fo.write('#include "{}/{}.h"\n'.format(args.package_name,"CoordinationSignal"))

if args.target_types:
    for x in args.target_types:
        msg=x.split(':')[0]
        fo.write('#include "{}/{}.h"\n'.format(args.package_name,msg))


#main
fo.write("\nint main(int argc, char *argv[]){\n")

# coordination signals
fo.write('CoordinationSignals<{}::CoordinationSignal>  coordSignals("coordination_signals");\n'.format(args.package_name))

# priority targets
priority_targets = []
if args.priority_topics:
    for x in args.priority_topics:
        topic=x.split(':')[0]
        msg_type=x.split(':')[1]
        fo.write("ReactiveInputs<{}::{}> {}(\"{}\");\n".format(args.package_name,msg_type,'prio_'+topic,topic))
        priority_targets.append('prio_'+topic)

fo.write('ResourceManager mgr(coordSignals, {});\n'.format('{'+', '.join(priority_targets)+'}'))


fo.write("}\n")

fo.close()
