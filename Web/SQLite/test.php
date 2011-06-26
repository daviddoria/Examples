<?php
try
{
  // open the database
  $database = new PDO('sqlite:test.db');
  $result = $database->query('SELECT * FROM Events');
  foreach($result as $row)
  {
    echo $row['EventName'] . ' | ' . $row['Location'];
  }
}
catch(Exception $e)
{
  die('Could not connect to database.');
}
?>