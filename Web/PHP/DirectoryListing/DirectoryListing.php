<html>
<body>
<?php
$dir = "Newsletters";

// Open a known directory, and proceed to read its contents
if (is_dir($dir)) {
    if ($dh = opendir($dir)) {
        while (($file = readdir($dh)) !== false) {
            echo "filename: $file : filetype: " . filetype($dir . $file) . "<br/>";
        }
        closedir($dh);
    }
}
?>
</body>
</html>