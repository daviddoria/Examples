import re
MyStr = "==some text here== ==some more text=="
print MyStr

m=re.compile('==(.*?)==').search(MyStr)

print "Group 0"
print m.group(0)

print "Group 1"
print m.group(1)

print "Group 2"
print m.group(2)
