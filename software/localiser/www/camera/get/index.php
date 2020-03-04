
<?php
error_reporting(E_ALL);

// Create a tcp request to 2115
$fp = fsockopen("127.0.0.1", 2115, $errno, $errstr, 30);
if (!$fp) {
    echo "$errstr ($errno)<br />\n";
} else {
    fwrite($fp, "1");
    while (!feof($fp)) {
        echo fgets($fp, 256);
    }
    fclose($fp);
}
?> 