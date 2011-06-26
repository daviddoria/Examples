<?php

$database = new PDO('sqlite:Test.db');

$myquery = 'SELECT * FROM TestTable';
$result = $database->query($myquery);
foreach($result as $row)
{
echo $row['Name'] . '<br/>';
}

?>