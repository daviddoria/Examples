<?php
include 'myfunction.php';

$returnedarray = myfunction();

for ($i = 0; $i < count($returnedarray); $i++)
{
    echo $returnedarray[$i];
}

?>