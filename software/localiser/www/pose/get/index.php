
<?php
error_reporting(E_ALL);

$sock = stream_socket_client('unix:///var/www/penguinpi/localiser.sock', $errno, $errstr);

if ($errno!=0){
    echo "Error creating socket: " . $errstr . "[" . $errno . "]"; 
}

fwrite($sock, '0'."\r\n");

$pose_response = fread($sock, 128);
socket_close($sock);

echo substr($pose_response,1,127);

?> 