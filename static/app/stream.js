'use strict';
define(['underscore', './consts'], function(_, consts)
{
    class Stream
    {
        /**
         * Default message handling function
         */
        _on_message(event)
        {
            _.each(this._handlers, function(handle) {
                handle(decodeURIComponent(event.data));
            });
        };

        _on_error()
        {
            console.log("Showing fancy error message.");
        };

        register(callback)
        {
            this._handlers.push(callback);
        };

        close()
        {
            this._ws.onclose = function() {};
            this._ws.close();
        };

        unregister(callback)
        {
            this._handlers = _.without(this._handlers, callback);
        };

        constructor(url)
        {
            this._url = url;
            this._ws = new WebSocket(consts.SOCKET_ADDRESS + this._url);
            this._ws.onmessage = this._on_message.bind(this);
            this._ws.onerror = this._on_error;
            this._handlers = [];
        };
    };

    return {
        Stream: Stream
    };
});
