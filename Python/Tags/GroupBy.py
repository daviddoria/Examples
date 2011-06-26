from itertools import groupby as gb
import re

S = """==Heading1==
<test>
some text here
</test> 

==Heading2==
<test>
even more text
</test>"""

GROUP = gb(S.splitlines(), lambda x: re.match("^==\w+==$$", x))
for k, g in GROUP:
    header = list(g)[0][2:-2]
    text = "\n".join(list(GROUP.next()[1])[1:-1])
   
    print "header".center(20, '-')
    print header
    print "text".center(20, '-')
    print text
