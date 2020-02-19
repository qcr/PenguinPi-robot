
<?php

header('Content-Type: text/html');

error_reporting(E_ALL);

$sock = stream_socket_client('unix:///var/www/penguinpi/localiser.sock', $errno, $errstr);

if ($errno!=0){
    echo "Error creating socket: " . $errstr . "[" . $errno . "]"; 
} else {
    // Request the localiser to s
    fwrite($sock, '1'."\r\n");

    $sock_response=fread($sock, 128);

    socket_close($sock);

    if ($sock_response[0] == '0') {

        echo "
        <html>
            <head>
            </head>
            <body> 
                <div> " . $sock_response . "</div>
                <div> <img src='/camera/arena.jpg' > </div>
            </body>
        </html>
        ";
    } else {

        echo "localiser error: " . $sock_response;
    }
}



?> 



