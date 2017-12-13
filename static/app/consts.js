'use strict';
define([], function()
{
    var WEB_ADDRESS = "http://" + document.location.hostname + ":8888/";
    var SOCKET_ADDRESS = "ws://" + document.location.hostname + ":8888/ws/";
    var ANGLE_OFFSET = -Math.PI / 2;
    var MAX_TICK = 255;

    return {
        WEB_ADDRESS: WEB_ADDRESS,
        SOCKET_ADDRESS: SOCKET_ADDRESS,
        ANGLE_OFFSET: ANGLE_OFFSET,
        MAX_TICK: MAX_TICK
    }
});
