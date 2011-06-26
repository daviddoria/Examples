<?php

$myarray[] = 1;
$myarray[] = 2;
$myarray[] = 2;
if(!in_array(3,$myarray)) { $myarray[] = 3; }
if(!in_array(2,$myarray)) { $myarray[] = 2; }

for ($i = 0; $i < count($myarray); $i++)
{
    echo $myarray[$i];
}

?>