// Make the DIV element draggable:


function dragElement(elmnt) {

    // Attach event listeners
    var pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;
    document.getElementById(elmnt.id).onmousedown = dragMouseDown;

    function dragMouseDown(e) {
      console.log("drag mouse down")
      e = e || window.event;
      e.preventDefault();
      // get the mouse cursor position at startup:
      pos3 = e.clientX;
      pos4 = e.clientY;
      document.onmouseup = closeDragElement;
      // call a function whenever the cursor moves:
      document.onmousemove = elementDrag;
    }
  
    function elementDrag(e) {
      console.log("element drag")
      e = e || window.event;
      e.preventDefault();
      // calculate the new cursor position:
      pos1 = pos3 - e.clientX;
      pos2 = pos4 - e.clientY;
      pos3 = e.clientX;
      pos4 = e.clientY;
      // set the element's new position:

      elmnt.style.top  = (elmnt.offsetTop - pos2) + "px";


      elmnt.style.left = (elmnt.offsetLeft - pos1) + "px";


      }
  
    function closeDragElement() {
      // stop moving when mouse button is released:
      document.onmouseup = null;
      document.onmousemove = null;

      var id = elmnt.id;
      var rect = elmnt.getBoundingClientRect();

      var new_x = rect.left;
      var new_y = rect.top;
 
      if (new_y > 479){
        new_y = 479;
        elmnt.style.top  = 479 + "px";

      }

      if (new_x > 639){
        new_x = 639;
        elmnt.style.left = 639 + "px";
      }

      console.log("tiepoint " + id + " at " + new_x + "," + new_y);
      $.post("/console/tiepoint_update.php",
      {
        corner: id,
        x: new_x,
        y: new_y
      },
      function(data, status){
        console.log("Data: " + data + "\nStatus: " + status);
      });
      
    }
  }

  $(document).ready( function() {

    // Attach event listeners to draggable markers 

    var tie_points = ["NW","NE","SE","SW"];

    for (var i=0; i< tie_points.length; i++){
        dragElement(document.getElementById(tie_points[i]));
    }
  });


