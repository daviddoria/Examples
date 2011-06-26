<?php
$xml = simplexml_load_file("bookstore.xml");

$res = $xml->xpath("/bookstore/book[title = 'Everyday Italian']"); 

echo $res[0]->author

?>