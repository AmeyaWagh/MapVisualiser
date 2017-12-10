'use strict';
define(['underscore', './utils', './consts.js'], function(_, utils, consts)
{
    function draw_bot_map(ctx, map)
    {
        for(var x = 0; x < map.num_cells_x; ++x)
        {
            for(var y = 0; y < map.num_cells_y; ++y)
            {
                var cell = map.cells[x][y];
                if(cell.ticks > 0)
                {
                    var ticks = cell.ticks;
                    var shading = utils.clamp(ticks, 0, 255);
                    var color = 'rgb(255' + ',' + (255 - shading) + ',' + (255 - shading) + ')';
                    ctx.fillStyle = color;
                    ctx.fillRect(cell.x, cell.y, cell.width, cell.height);
                }
            }
        }
    }

    class Renderer
    {
        constructor(canvas)
        {
            this.canvas = canvas;
            this.ctx = this.canvas.getContext('2d');
            this.best_networks = [];
            this.best_nn_image_1 = new Image();
            this.best_nn_image_2 = new Image();
            this.best_nn_image_3 = new Image();
            this.best_nn_image_4 = new Image();
            this.selected_network = new Image();
        }

        init_graphics()
        {
            this.canvas.style.position = "absolute";
        }

        draw_border()
        {
            this.ctx.strokeStyle = 'blue';
            this.ctx.lineWidth = 2;
            this.ctx.strokeRect(0, 0, this.canvas.width, this.canvas.height);
        }

        draw_bot(bot_frame, image)
        {
            this.ctx.save();  /// SAVE

            this.ctx.beginPath();
            var x = bot_frame[0];
            var y = bot_frame[1];
            var rotation = bot_frame[2];

            var x_offset = image.width / 2;
            var y_offset = image.height / 2;
            this.ctx.translate(x, y);
            this.ctx.rotate(rotation + consts.ANGLE_OFFSET);
            this.ctx.drawImage(image, -x_offset, -y_offset, image.width, image.height);

            this.ctx.restore();  /// RESTORE

            this.ctx.fillStyle = 'red';
            utils.draw_circle(this.ctx, x, y, 2);
        }

        draw_bot_sensors(bot_frame, sensors)
        {
            this.ctx.fillStyle = 'blue';
            var x = bot_frame[0];
            var y = bot_frame[1];
            var radius = 2;

            for(var i = 0; i < sensors.length; ++i)
            {
                var sensor = sensors[i];

                this.ctx.fillStyle = 'blue';
                this.ctx.strokeStyle = 'blue';

                this.ctx.beginPath();
                this.ctx.moveTo(x, y);
                this.ctx.lineTo(sensor[0], sensor[1]);
                this.ctx.stroke();
                utils.draw_circle(this.ctx, sensor[0], sensor[1], radius);
            }
        }

        clear_canvas()
        {
            this.ctx.save();
            this.ctx.beginPath();
            this.ctx.setTransform(1, 0, 0, 1, 0, 0);
            this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
            this.ctx.restore();
        }

        render(bots)
        {
            this.clear_canvas();
            this.draw_bots(bots);
            this.draw_border();
        }
    };

    return {
        Renderer: Renderer
    }
});
