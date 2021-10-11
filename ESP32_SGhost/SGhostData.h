
// web page content
const char* index_html = R"~(
<!doctype html>
<html>
  <head>
    <meta http-equiv="Content-Type" content="text/html; charset=utf-8" />
    <title>ESP32_SGhost</title>
    <script src="https://ajax.googleapis.com/ajax/libs/jquery/2.1.3/jquery.min.js"></script>
    <STYLE type="text/css">
      body {
        margin: 10px;
      }
    
      table {
        table-layout: fixed;
        font-family: arial, sans-serif;
        border-collapse: collapse;
        width: 700px;
      }
        
      td, th {
        border: 1px solid #dddddd;
        text-align: left;
        padding: 8px;
      }
      
      tr:nth-child(even) {
        background-color: #dddddd;
      } 
      input[type="text"] {
        font-size: 16px; 
        width: 100px
      }
      input[type="submit"], input[type="file"] {
        padding: 10px; 
        background-color: #e7e7e7;
        font-size: 16px; 
      }
      input:active{ background-color:green}
    </STYLE>
  </head>
  <body>
    <table><tr>
      <td>Time Stamp:</td>
      <td id="0">Waiting</td>
      </tr><tr>
      <th>Specific Gravity</th>
      <th>Temperature</th>
      </tr><tr>
      <td id="2">Waiting</td>
      <td id="5">Waiting</td>
      </tr><tr>
      <th>Tilt Angle</th>
      <th>Voltage</th>
      </tr><tr>
      <td id="1">Waiting</td>
      <td id="4">Waiting</td>
      </tr>
    </table>
    <script>
      var baseHost = document.location.origin
      var myUrl = baseHost +"/";
      var refreshRate = 2000; // 2 seconds
      var refresh = true;
      $(function(){updatePage();}); 
    
      function updatePage(){ 
        if (refresh) {
          console.log("updatePage");   
          // periodically update page content using received JSON
          var myData = $.ajax({ // send request to app
            url: myUrl+"refresh",           
            dataType : "json", 
            timeout : refreshRate, 
            success: function(data) { // receive response from app
              $.each(data, function(key, val) { 
                // replace each existing value with new value, using key name to match html tag id
                $('#'+key).text(val);
                $('#'+key).val(val);
              });
            }
          });
              
          myData.error(function(xhr, status, errorThrown){ 
            console.log("Failed to get data: " + errorThrown); 
            console.log("Status: " + status);
            console.dir(xhr);
          });
        }
        timeOut = setTimeout('updatePage()', refreshRate);  // re-request data at refreshRate interval in ms
      }
    </script>
  </body>
</html>
)~";

