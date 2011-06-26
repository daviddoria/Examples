import re

def parse_wiki(S):
    reg = re.compile(r"==([\w]*?)==.*?<test>(.*?)</test>.*?", re.DOTALL)
    return reg.findall(S)


S = """==HeadingWithoutSpace==
Some intro text.
<other>
blah blah blah
</other>
<test>
some text here
</test> 
<other>
blah blah blah
</other>

==Heading With Space==
More intro text.
<test>
even more text
and even more text
</test>"""

a = parse_wiki(S)
a = [(x.strip('\n'), y.strip('\n')) for (x,y) in a]
print a

#for i in a
#	i.strip('\n')
#	print i

