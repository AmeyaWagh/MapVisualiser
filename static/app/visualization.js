define(['underscore', './map', './graphics', './stream', './utils'], function(_, map, graphics, stream, utils)
{
    class Visualization
    {
        constructor(canvas_id)
        {
            this.sensor_stream = new stream.Stream("sensors");
            this.sensor_stream.register(this.process_sensor_data.bind(this));

            this.imu_stream = new stream.Stream("imu_data")
            this.imu_stream.register(this.save_imu_data.bind(this));

            this.image = new Image();
            this.image.src = "./static/images/tank.png";

            this.canvas = document.getElementById(canvas_id);

            this.renderer = new graphics.Renderer(this.canvas);
            this.setup_events();

            this.renderer.init_graphics();
            this.renderer.draw_border();
            this.renderer.draw_info_menu();
            this.fps_counter = new utils.FPSCounter();
            this.map = new map.Map(this.canvas.width, this.canvas.height, 10);
            this.imu_data = null;
        }

        save_imu_data(imu_data)
        {
            this.imu_data = JSON.parse(imu_data);
        }

        process_sensor_data(sensors)
        {
            var data = JSON.parse(sensors);

            if(_.isNull(data) || _.isUndefined(data))
            {
                return;
            }

//            _.each(data.sensorValues, function(position, idx, list){
//                try
//                {
//                    if(position[0] != 0 && position[1] != 0)
//                    {
//                        this.map.update(position[0], position[1]);
//                    }
//                }
//                catch(exception)
//                {
//                    console.log("Negative indices, why?");
//                }
//
//            }.bind(this));

            this.renderer.clear_canvas();
            this.fps_counter.count_frame();
//            this.renderer.draw_bot_map(this.map);
            this.renderer.draw_border();
            this.renderer.draw_bot(data.frame, this.image);
            this.renderer.draw_bot_sensors(data.frame, data.sensorValues);
            this.renderer.draw_info_menu(this.imu_data);
        }

        setup_events()
        {
            document.addEventListener("keydown", this.handle_key_event.bind(this));
        }

        handle_key_event(evnt)
        {
            console.log(evnt.key);
        }
    }

    return {
        Visualization: Visualization
    };
});
