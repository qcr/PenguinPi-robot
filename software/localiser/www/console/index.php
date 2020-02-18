<?php 


    header('Content-Type: text/html');

    error_reporting(E_ALL);

    $sock = stream_socket_client('unix:///var/run/penguinpi/localiser.sock', $errno, $errstr);

    if ($errno!=0){
        echo "Error creating socket: " . $errstr . "[" . $errno . "]"; 
    }

    fwrite($sock, '2'."\r\n");
    $socket_response = fread($sock, 256)."\n";
    socket_close($sock);

    $sock = stream_socket_client('unix:///var/run/penguinpi/localiser.sock', $errno, $errstr);

    if ($errno!=0){
        echo "Error creating socket: " . $errstr . "[" . $errno . "]"; 
    }

    fwrite($sock, '3'."\r\n");
    $socket_response = fread($sock, 256)."\n";
    socket_close($sock);

    $tie_points = json_decode(trim($socket_response), true);
    
    $tie_point_html = "";

    foreach($tie_points as $key => $value){
        $tie_point_html .= "<div class=\"tie_point\" id=\"" . $key ."\" style=\"top:". $value['y'] . "px; left: " . $value['x'] . "px;\"><img src=\"icon.png\"></div>";
    }

?>

<!doctype html>

<html>
    <head>
        <script type="text/javascript" src="jquery-3.4.1.min.js"></script>
        <script type="text/javascript" src="console.js"></script>
        <link rel="stylesheet" type="text/css" href="style.css">
    </head>

    <body>

    <div id="arena_image"> 
        <img src="/camera/camera_raw.jpg">
        <div> 
            <?php echo $tie_point_html; ?>
        </div>
    </div>
    <div class="slidecontainer">
  <input type="range" min="100" max="2000" value="100" class="slider" id="shutterSpeed">
    </div>
    <div>
    <button id ="SSbutton" type="button">Update shutter speed</button> 
    </div>
    </body>
</html>


