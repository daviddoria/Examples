<?php
$xml = simplexml_load_file("bookstore.xml");

foreach($xml->children() as $child)
  {
  //echo $child->title . "<br/>";
  echo "<a href=bookdetails.php?title=" . $child->title . ">" . $child->title . "</a> <br/>";
  }
?>