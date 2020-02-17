
<?php

header('Content-Type: text/html');

error_reporting(E_ALL);

$sock = stream_socket_client('unix:///var/run/penguinpi/localiser.sock', $errno, $errstr);

if ($errno!=0){
    echo "Error creating socket: " . $errstr . "[" . $errno . "]"; 
}

// Request the localiser to s
fwrite($sock, '1'."\r\n");

$sock_response=fread($sock, 128)."\n";

socket_close($sock);

$imgpath = '/camera/arena.jpg';

echo "
<html>
    <head>
    </head>
    <body>
        <img src='". $imgpath . "' >
    </body>
</html>
"

?> 

