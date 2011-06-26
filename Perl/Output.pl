#!/usr/local/bin/perl
#
# Program to do the obvious
#
print 'Hello world.';		# Print a message

$a = 'apples';
$b = 'pears';

print $a.' and '.$b;
print '$a and $b';
print "$a and $b";


for ($i = 0; $i < 10; ++$i)	# Start with $i = 1
				# Do it while $i < 10
				# Increment $i before repeating
{
	print "$i\n";
}

for ($i = 0; $i < 10; ++$i)	# Start with $i = 1
				# Do it while $i < 10
				# Increment $i before repeating
{
	print "$i hello\n";
	print "${i}hello\n";
}

