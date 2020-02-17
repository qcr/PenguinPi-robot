
<?php
error_reporting(E_ALL);

$sock = stream_socket_client('unix:///var/run/penguinpi/localiser.sock', $errno, $errstr);

if ($errno!=0){
    echo "Error creating socket: " . $errstr . "[" . $errno . "]"; 
}

fwrite($sock, '0'."\r\n");

echo fread($sock, 128)."\n";

socket_close($sock);

?> 