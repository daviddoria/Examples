<html>
<body>

<?php
try
{
  $database = new PDO('sqlite:Test.db');
  $myquery = 'select * from TestTable Order By DateColumn';
  $result = $database->query($myquery);

  foreach($result as $row)
  {
    echo $row['DateColumn'] . '<br/>';
    echo date("Y", strtotime($row['DateColumn'])) . '<br/>'; // year (i.e. 2010)
    echo date("M", strtotime($row['DateColumn'])) . '<br/>'; // short month (i.e. Dec)
    echo date("F", strtotime($row['DateColumn'])) . '<br/>'; // long month (i.e. December)
    echo date("l", strtotime($row['DateColumn'])) . '<br/>'; // day of week (i.e. Wednesday)
  }

/*
  $MyDate = '2010-10-02';
  echo date("Y", strtotime($MyDate)) . '<br/>'; // shows 1969
  echo date("F", strtotime($MyDate)) . '<br/>'; // shows December
*/
}
catch(Exception $e)
{
  die('Could not connect to database.');
}
?>

</body>
</html>
