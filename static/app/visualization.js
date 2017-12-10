define(['underscore', './map', './graphics', './stream', './utils'], function(_, map, graphics, stream, utils)
{
    class Visualization
    {
        constructor(canvas_id)
        {
            this.sensor_stream = new stream.Stream("sensors");
            this.sensor_stream.register(this.process_sensor_data.bind(this));

            this.image = new Image();
            this.image.src = "./static/images/tank.png";

            this.canvas = document.getElementById(canvas_id);

            this.renderer = new graphics.Renderer(this.canvas);

            this.renderer.init_graphics();
            this.renderer.draw_border();
            this._fps_counter = new utils.FPSCounter();
        }

        process_sensor_data(sensors)
        {
            this.renderer.clear_canvas();
            var data = JSON.parse(sensors);
            this._fps_counter.count_frame();
            this.renderer.draw_border();
            this.renderer.draw_bot(data.frame, this.image);
            this.renderer.draw_bot_sensors(data.frame, data.sensorValues)
        }
    }

    return {
        Visualization: Visualization
    };
});
