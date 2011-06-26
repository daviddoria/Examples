import re
#MyStr = "<test>some text here</test> <other> more text </other> <test> even more text</test>"
MyStr = "==some text here=="
m=re.compile('<test>(.*?)</test>').search(MyStr)
print m.group(1)
print m.group(2)