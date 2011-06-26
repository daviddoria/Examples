<?php
// Great tutorial : http://www.phpro.org/tutorials/Introduction-to-PHP-PDO.html#1

$database = new PDO('sqlite:Test.db');
$myquery = 'SELECT * FROM TestTable';

/*
// get one row
$statement = $database->query($myquery);
$result = $statement->fetch(PDO::FETCH_ASSOC);

foreach($result as $key=>$val)
{
echo $key.' - '.$val.'<br />';
}
*/

// Get all rows
/*
$statement = $database->query($myquery);
$fetchResult = $statement->FetchAll(PDO::FETCH_ASSOC);

foreach($fetchResult as $result)
{
  echo '<br/>';
  foreach($result as $key=>$val)
  {
  echo $key.' - '.$val.'<br />';
  }
}
*/

// Object oriented method, one row
/*
echo 'Object oriented:' . '<br/>';
$statement = $database->query($myquery);
$fetchResult = $statement->fetch(PDO::FETCH_OBJ);
echo $fetchResult->Name . '<br />'; // Case sensitive!
*/

// Object oriented method, all rows
echo 'Object oriented:' . '<br/>';
$statement = $database->query($myquery);
$fetchResult = $statement->FetchAll(PDO::FETCH_OBJ);
foreach($fetchResult as $result)
{
echo $result->Name . '<br />'; // Case sensitive!
}


?>