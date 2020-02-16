
<?php
error_reporting(E_ALL);

// if(!($sock = socket_create(AF_UNIX, SOCK_STREAM, 0)))
// {
//     $errorcode = socket_last_error();
//     $errormsg = socket_strerror($errorcode);

//     die("Couldn't create socket: [$errorcode] $errormsg \n");
// }

$sock = stream_socket_client('unix:///var/run/penguinpi/localiser.sock', $errno, $errstr);
// {
//     $errorcode = socket_last_error();
//     $errormsg = socket_strerror($errorcode);
//     die("Could not connect: [$errorcode] $errormsg \n");
// }

if ($errno!=0){
    echo "Error creating socket: " . $errstr . "[" . $errno . "]"; 
}


fwrite($sock, '0'."\r\n");

echo fread($sock, 128)."\n";

socket_close($sock);

?> 