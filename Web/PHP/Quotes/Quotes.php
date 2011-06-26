<?php

$TableName = 'TestTable';
$Name = 'TestName';

echo '<input type="hidden" value="' . 'SELECT * FROM ' . $TableName . ' WHERE Name=\'' . $Name . '\'" name="query">';

?>
