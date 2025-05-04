var joyLeftX;
var joyLeftY;
var joyRightX;
var joyRightY;

let socket = null;

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

document.getElementById("test-tcs").addEventListener("click", function () {
    if (socket) {
        socket.send(
            JSON.stringify({
                type: "command",
                payload: {
                    test: 1,
                },
            }),
        );
    }
});

function sendControls() {
    var armingList = [];
    document
        .querySelectorAll(".arming")
        .forEach((cb) => armingList.push(cb.checked ? 1 : 0));

    if (socket) {
        socket.send(
            JSON.stringify({
                type: "command",
                payload: {
                    joysticks: {
                        left: [parseInt(joyLeftX), parseFloat(joyLeftY)],
                        right: [parseInt(joyRightX), parseFloat(joyRightY)],
                    },
                    arming: armingList,
                },
            }),
        );
    }
}

setInterval(sendControls, 300);

document.getElementById("connect").addEventListener("click", function () {
    if (document.getElementById("password").value == "") {
        return;
    }
    if (socket && socket.readyState === WebSocket.OPEN) {
        console.log("[WS] Already connected.");
        return;
    }

    socket = new WebSocket("wss://chillsam.ddns.net:8012/echo");

    socket.addEventListener("open", () => {
        console.log("[WS] Connected");
    });

    socket.addEventListener("message", async (event) => {
        const msg = JSON.parse(event.data);

        if (msg.nonce) {
            console.log("[WS] Received nonce:", msg.nonce);
            const authPayload = await buildAuthPayload(
                "controller",
                msg.nonce,
                document.getElementById("password").value,
            );
            socket.send(JSON.stringify(authPayload));
        } else {
            console.log("[WS] Message from server:", msg);
        }
    });

    socket.addEventListener("close", () => {
        console.log("[WS] Connection closed");
    });

    async function buildAuthPayload(role, nonce, secret) {
        const enc = new TextEncoder();
        const keyData = enc.encode(secret);
        const msgData = enc.encode(nonce);

        const cryptoKey = await crypto.subtle.importKey(
            "raw",
            keyData,
            { name: "HMAC", hash: "SHA-256" },
            false,
            ["sign"],
        );

        const sigBuffer = await crypto.subtle.sign("HMAC", cryptoKey, msgData);
        const sigHex = bufferToHex(sigBuffer);

        return {
            role,
            signature: sigHex,
        };
    }

    function bufferToHex(buffer) {
        const bytes = new Uint8Array(buffer);
        return Array.from(bytes)
            .map((b) => b.toString(16).padStart(2, "0"))
            .join("");
    }
});

function tune(axis, tuningData) {
    socket.send(
        JSON.stringify({
            type: "pid",
            payload: { axis: axis, tuningData: tuningData },
        }),
    );
}
