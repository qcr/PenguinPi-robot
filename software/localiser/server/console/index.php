<?php 


    header('Content-Type: text/html');

    error_reporting(E_ALL);

    $sock = stream_socket_client('unix:///var/run/penguinpi/localiser.sock', $errno, $errstr);

    if ($errno!=0){
        echo "Error creating socket: " . $errstr . "[" . $errno . "]"; 
    }

    fwrite($sock, '2'."\r\n");
    $socket_response = fread($sock, 128)."\n";
    socket_close($sock);

    $string = file_get_contents("tie_points.json");
    $tie_points = json_decode($string, true); 

    $tie_point_html = "";

    foreach($tie_points as $key => $value){
        $tie_point_html .= "<div class=\"tie_point\" id=\"" . $key ."\" style=\"top:". $value['y'] . "px; left: " . $value['x'] . "px;\"><img src=\"icon.png\"></div>";
    }

?>

<!doctype html>

<html>
    <head>
        <script type="text/javascript" src="jquery-3.4.1.min.js"></script>
        <script type="text/javascript" src="drag.js"></script>
        <link rel="stylesheet" type="text/css" href="style.css">
    </head>

    <body>

    <div id="arena_image"> 
        <img src="/camera/camera_raw.jpg">
        <div> 
            <?php echo $tie_point_html; ?>
        </div>
    </div>
    </body>
</html>


