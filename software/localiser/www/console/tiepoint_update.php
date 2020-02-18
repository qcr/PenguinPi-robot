<?php
   if( $_POST["corner"] && $_POST["x"] && $_POST["y"]) {

    $sock = stream_socket_client('unix:///var/www/penguinpi/localiser.sock', $errno, $errstr);

        if ($errno!=0){
            echo "Error creating socket: " . $errstr . "[" . $errno . "]"; 
        }

        fwrite($sock, '2'."\r\n");
        $socket_response = fread($sock, 256)."\n";
        socket_close($sock);

        $sock = stream_socket_client('unix:///var/www/penguinpi/localiser.sock', $errno, $errstr);

        if ($errno!=0){
            echo "Error creating socket: " . $errstr . "[" . $errno . "]"; 
        }

        $tiepoint_id = -1;
        switch ($_POST["corner"]) {
            case "NW":
                $tiepoint_id = 0;
                break;
            case "NE":
                $tiepoint_id = 1;
                break;
            case "SE":
                $tiepoint_id = 2;
                break;
            case "SW":
                $tiepoint_id = 3;
            default:
                echo "Invalid tiepoint ID: " . $_POST["corner"] ;
        }

        $localiser_request = sprintf("4%1.d%4.d%4.d",$tiepoint_id, $_POST["x"], $_POST["y"]);

        fwrite($sock, $localiser_request."\r\n");
        $socket_response = fread($sock, 256)."\n";
        socket_close($sock);

        echo "Sent request to localiser " . $localiser_request;
   } else {
       echo "Incorrectly formed request";
   }
?>