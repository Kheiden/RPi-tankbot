function start_left_cam_stream()
{
var img = document.getElementById("left_cam_stream");
img.src="http://192.168.1.16:5000/left_camera_stream";
log_to_console("Starting left camera stream")
return false;
}
data = 'data'

function log_to_console(text)
{
    d = new Date();
    document.getElementById('console').innerHTML = 
        document.getElementById('console').innerHTML
        + d.getFullYear()
        + "-" + (d.getMonth() + 1) + "-" + d.getDay()
        + " | "
        + d.getHours() + ":" + d.getMinutes() + ":" + d.getSeconds() + "." + d.getMilliseconds()
        + " | "
        + "INFO | "
        + text + "<br \>"
}

function test()
{
    document.getElementById('console').innerHTML = "test"
    return
}