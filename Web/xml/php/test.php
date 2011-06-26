<?php
$xml = simplexml_load_file("test.xml");

echo $xml->getName() . "<br />";

foreach($xml->children() as $child)
  {
  foreach($child->children() as $subchild)
    {
    echo $subchild->getName() . ": " . $subchild . "<br />";
    }
  }
?>