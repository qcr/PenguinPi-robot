<?php 


    header('Content-Type: text/html');

    error_reporting(E_ALL);

    $sock = stream_socket_client('unix:///var/www/penguinpi/localiser.sock', $errno, $errstr);

    if ($errno!=0){
        echo "Error creating socket: " . $errstr . "[" . $errno . "]"; 
    }

    fwrite($sock, '2'."\r\n");
    $socket_response = fread($sock, 256);

    if ($socket_response[0]=='0'){
        $image_html = "<img src='/camera/camera_raw.jpg'>";
    } else {    
        $image_html = "<span>Localiser error</span>";
    }
    socket_close($sock);
    
    $tie_point_html = "";

    $sock = stream_socket_client('unix:///var/www/penguinpi/localiser.sock', $errno, $errstr);
    if ($errno!=0){
        echo "Error creating socket: " . $errstr . "[" . $errno . "]"; 
    }

    fwrite($sock, '3'."\r\n");
    $tiepoint_response = fread($sock, 256);

    vardump($tiepoint_response);
    //if ($tiepoint_response[0] == '0'){
        //$tie_points = json_decode(trim(substr($tiepoint_response,1,200)), true);
        
        // foreach($tie_points as $key => $value){
        //     $tie_point_html .= "<div class=\"tie_point\" id=\"" . $key ."\" style=\"top:". $value['y'] . "px; left: " . $value['x'] . "px;\"><img src=\"icon.png\"></div>";
        // }
    //} 

    socket_close($sock);

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
        <?php echo $image_html; ?>
        <div> 
            <?php echo $tie_points; ?>
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


