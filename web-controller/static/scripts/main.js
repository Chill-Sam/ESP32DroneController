var joyLeftX;
var joyLeftY;
var joyRightX;
var joyRightY;

var joyLeft = new JoyStick("JoystickLeft", {}, function (stickdata) {
    joyLeftX = stickdata.x;
    joyLeftY = stickdata.y;
});

var joyRight = new JoyStick("JoystickRight", {}, function (stickdata) {
    joyRightX = stickdata.x;
    joyRightY = stickdata.y;
});

document.getElementById("arm-fcu").addEventListener("change", function () {
    const isChecked = this.checked;
    const checkboxes = document.querySelectorAll(".arm-motor");

    checkboxes.forEach((cb) => {
        cb.disabled = !isChecked;
        if (!isChecked) cb.checked = false;
    });
});

document.querySelectorAll(".arming").forEach((cb) => {
    cb.addEventListener("change", function () {
        console.log("Hello World");
    });
});
