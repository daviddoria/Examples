#!/usr/bin/python

#known length:
print "%08d" % 2
# prints '00000002'

#assign
a="%08d" % 2
print a

#unknown length
MyLength = 3
print eval("'%0" + str(MyLength) + "d' % 2")
a=eval("'%0" + str(MyLength) + "d' % 2")
print a

# OR
NewLength = 4
temp = "%0" + str(NewLength) + "d"
temp2 = temp % 2
print temp2
