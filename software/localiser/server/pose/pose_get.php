
<?php
error_reporting(E_ALL);

if(!($sock = socket_create(AF_UNIX, SOCK_STREAM, 0)))
{
    $errorcode = socket_last_error();
    $errormsg = socket_strerror($errorcode);

    die("Couldn't create socket: [$errorcode] $errormsg \n");
}

echo "Socket created";

if(!socket_connect($sock,"/tmp/server.sock"))
{
    $errorcode = socket_last_error();
    $errormsg = socket_strerror($errorcode);
    die("Could not connect: [$errorcode] $errormsg \n");
}

echo "Connection established \n";

$message = "hi from PHP";

if(!socket_send( $sock , $message , strlen($message) , 0))
{
    $errorcode = socket_last_error();
    $errormsg = socket_strerror($errorcode);

    die("Could not send data: [$errorcode] $errormsg \n");
}

echo "Message send successfully \n";

// Now receive reply from server
if(socket_recv( $sock , $buf , 1024, MSG_WAITALL ) === FALSE)
{
    $errorcode = socket_last_error();
    $errormsg = socket_strerror($errorcode);
    die("Could not receive data: [$errorcode] $errormsg \n");
}

echo "Message received \n";

// print the received message
var_dump($buf);

socket_close($sock);

?> 