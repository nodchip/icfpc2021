const SERVER_URL = "";
const canvas = $("#picture");
const canvas_context = $("#picture").get(0).getContext('2d');
const W = canvas.width(); // pixel
const H = canvas.height(); // pixel
const background_color = "rgb(0, 0, 0)";
const grid_color = "rgb(32, 32, 32)";
var enable_grid = true;
var scale = 3;
const logicalW = W / scale;
const logicalH = H / scale;
const offsetx = - W / 2; // pixel
const offsety = - H / 2; // pixel
var pixels = {};

const pixel2logical = function (x, y) {
    return [Math.floor((x + offsetx) / scale), Math.floor((y + offsety) / scale)];
};
const logical2pixel = function (x, y) {
    return [x * scale - offsetx, y * scale - offsety];
};

const append_text_multiselect_OLD = function (data) {
    const option = $("<option>").val(data).text(data);
    $("#eval-result").append(option);
    $("#eval-result").scrollTop(option.position().top);
};
const append_text = function (data) {
    const console = $('#eval-result');
    console.text(
        console.text() + data + "\n"
    );
    if (console.length) {
        console.scrollTop(console[0].scrollHeight - console.height());
    }
};

const clear_canvas = function () {
    //console.log("clear");
    canvas_context.fillStyle = "rgb(0, 0, 0)";
    canvas_context.clearRect(0, 0, W, H);
    canvas_context.rect(0, 0, W, H);
    canvas_context.fill();
};

const draw_grid_on_canvas = function () {
    // too fine to show grids.
    if (!enable_grid || scale < 8) return;

    canvas_context.strokeStyle = grid_color;
    canvas_context.lineWidth = 1;
    const fy = Math.floor(- H / scale / 2);
    const ty = Math.ceil(H / scale / 2);
    const fx = Math.floor(- W / scale / 2);
    const tx = Math.ceil(W / scale / 2);
    for (var ly = fy; ly < ty; ++ly) {
        const [px_begin, py] = logical2pixel(fx, ly);
        const [px_end, _   ] = logical2pixel(tx, ly);
        canvas_context.beginPath();
        canvas_context.moveTo(px_begin, py);
        canvas_context.lineTo(px_end, py);
        canvas_context.closePath();
        canvas_context.stroke();
    }
    for (var lx = fx; lx < tx; ++lx) {
        const [px, py_begin] = logical2pixel(lx, fy);
        const [_ , py_end  ] = logical2pixel(lx, ty);
        canvas_context.beginPath();
        canvas_context.moveTo(px, py_begin);
        canvas_context.lineTo(px, py_end);
        canvas_context.closePath();
        canvas_context.stroke();
    }
};

const update_status_waiting = function () {
    $("#status-waiting").show();
    $("#status-ready").hide();
};
const update_status_ready = function () {
    $("#status-waiting").hide();
    $("#status-ready").show();
};

const draw_pixels = function (pixels_) {
    clear_canvas();
    for (const [x, row] of Object.entries(pixels_)) {
        for (const [y, rgb] of Object.entries(row)) {
            const [px, py] = logical2pixel(x, y);
            //console.log(x, y, rgb);
            canvas_context.fillStyle = "rgb(" + rgb[0] + "," + rgb[1] + "," + rgb[2] + ")";
            canvas_context.fillRect(px, py, scale, scale);
        }
    }
    draw_grid_on_canvas();
};

const update_node_statistics = function () {
    $.getJSON(SERVER_URL + "/node_statistics", {}, function (j) {
        const $node_stat = $("#node-stat");
        $node_stat.empty();
        const name_count = [];
        const sort_stat_by_name = $("#btn-sort-stat-by-name").hasClass("active");
        for (var [name, count] of Object.entries(j)) {
            name = name.toLowerCase().replace(/struct /, ""); // Windows: typeid(type).name() => "struct Node"
            if (sort_stat_by_name) {
                name_count.push([name, [name, count]]);
            } else {
                name_count.push([-count, [name, count]]); // descending
            }
        }
        name_count.sort((lhs, rhs) => {
            if (lhs[0] > rhs[0]) return 1;
            if (lhs[0] < rhs[0]) return -1;
            return 0;
        });
        //console.log(sort_stat_by_name, name_count);
        name_count.forEach(function ([key, [name, count]]) {
            //console.log(name, count);
            $node_stat.append($('<button type="button" class="btn btn-outline-info btn-sm"></button>').text(name + ":" + count));
        });
    });
};

const color_unit = (age, size) => {
    const pattern = [255, 128, 100, 90, 80, 70, 60, 50];
    if (age >= pattern.length)
	return 30;
    return pattern[age];
};

const update_picture = function (picture_size) {
    $.post(SERVER_URL + "/picture", JSON.stringify({ "unused" : 0 }), function (jpicture) {
        //console.log("picture", jpicture);
        if (jpicture["picture"].length > 0) {
	    // Draw at most 10 pictures
	    if (jpicture["picture"].length > 10) {
		console.log("skip some pictures", jpicture);
	    }
	    pictures = jpicture["picture"].slice(-10);

	    const size = pictures.length;
            pixels = {}; // (x, y) => color
            pictures.forEach(function (single_channel_pixels, age) {
		const unit = color_unit(age, size);
                single_channel_pixels.forEach(function (xy) {
                    const [x, y] = xy;
                    //console.log("xy", x, y, age);
                    if (pixels[x] === undefined) {
                        pixels[x] = {};
                    }
                    if (pixels[x][y] === undefined) {
                        pixels[x][y] = [0, 0, 0];
                    }
                    pixels[x][y][0] += unit;
                    pixels[x][y][1] += unit;
                    pixels[x][y][2] += unit;
                })
            });
            //console.log(pixels);
        }

        draw_pixels(pixels);
    }, "json");
};

const communicate = function (endpoint_path, text_to_senddata, text, callback) {
    update_status_waiting();
    $.get(SERVER_URL + endpoint_path, text_to_senddata(text), function (data) {
        //console.log("RECV " + data);
        if (callback) {
            callback(data);
        }
        data.split("\n").forEach(function (line) {
            if (line.length) {
                append_text(line);
            }
        });
        // maybe updated..
        $.getJSON(SERVER_URL + "/picture_size", {}, function (j) {
            const size = j["picture_history_size"];
            if (size > 0) {
                update_picture(size);
            }
        });
        // and update stats.
        update_node_statistics();
        update_status_ready();
    });
};
const process_parse_expr = function (text, callback) {
    communicate("/eval", stmt => { return { "expr": expr_text }; }, text, callback);
};
const process_parse_statement = function (text, callback) {
    communicate("/parse_statement", stmt => { return { "stmt": stmt }; }, text, callback);
};
const process_click = function (logicalx, logicaly) {
    update_status_waiting();
    $.getJSON(SERVER_URL + "/click", { "x": logicalx, "y": logicaly }, function (j) {
        //console.log(j);
        const picture_size = j["picture_history_size"];
        update_picture(picture_size);
        // and update stats.
        update_node_statistics();
        update_status_ready();
    });
};

const set_key_handler = function (obj, endpoint_path, text_to_senddata) {
    const KEY_ENTER = 13;
    const KEY_UP = 38;
    const KEY_DOWN = 40;
    const history = [""];
    var history_index = 0;
    const update_history = function (text) {
        const i = history.findIndex(v => (v === text));
        if (i !== -1) {
            history.splice(i, 1);
        }
        history.unshift(text);
        history_index = 0;
        //console.log("UPD", history, history_index);
    };
    obj.keypress(function (e) {
        var code = (e.keyCode ? e.keyCode : e.which);
        if (code == KEY_ENTER) {
            const text = $(this).val();
            $(this).val("");
            $(this).attr("placeholder", ""); // no longer needed..
            //console.log("SEND " + text);
            append_text("> " + text);
            update_history(text);
            communicate(endpoint_path, text_to_senddata, text);
            return false;
        }
    });
    obj.keydown(function (e) {
        var code = (e.keyCode ? e.keyCode : e.which);
        switch (code) {
            case KEY_UP:
                //console.log("UP", history, history_index);
                $(this).val(history[history_index]);
                history_index = Math.min(history_index + 1, history.length);
                e.preventDefault();
                return false;

            case 40:
                //console.log("DOWN", history, history_index);
                $(this).val(history[history_index]);
                history_index = Math.max(history_index - 1, 0);
                e.preventDefault();
                return false;
        }
    });
};

const get_logical_xy = function (ev) {
    const rect = canvas.offset();
    return pixel2logical(
        Number.parseInt(ev.pageX - rect.left),
        Number.parseInt(ev.pageY - rect.top));
};

$(function () {
    $("form").submit(function (e) {
        return false;
    });
    set_key_handler($("#eval-input"), "/eval", stmt => { return { "expr": expr_text }; });
    set_key_handler($("#stmt-input"), "/parse_statement", stmt => { return { "stmt": stmt }; });

    const on_scale_slider = function (ev, ui) {
        scale = ui.value;
        $("#scale-slider-value").text(ui.value);
        draw_pixels(pixels);
    };
    $("#scale-slider").slider({
        min: 1,
        max: 30,
        slide: on_scale_slider,
        change: on_scale_slider,
    });
    $("#scale-slider").slider("value", 3);

    canvas.on("mousemove", function (ev) {
        const [logicalx, logicaly] = get_logical_xy(ev);
        $("#logical-x").text(logicalx);
        $("#logical-y").text(logicaly);
    });

    // toolbar
    $("#btn-click-0-0").click(function () {
        append_text("clicking (0, 0)");
        process_click(0, 0);
    });
    $("#btn-sort-stat-by-name").click(function () {
        if ($(this).hasClass("active")) {
            $(this).removeClass("active");
        } else {
            $(this).addClass("active");
        }
        update_node_statistics();
    });
    $("#btn-grid").click(function () {
        if ($(this).hasClass("active")) {
            enable_grid = false;
            clear_canvas();
            draw_pixels(pixels);
            $(this).removeClass("active");
        } else {
            enable_grid = true;
            draw_grid_on_canvas();
            $(this).addClass("active");
        }
    });
    $("#btn-setup-galaxy").click(function () {
        process_parse_statement("!reset_galaxy", function () {
            process_parse_statement("!startup_galaxy");
        });
    });
    $("#btn-reset-galaxy").click(function () {
        process_parse_statement("!reset_galaxy");
    });
    // initialize special commands dropdown.
    process_parse_statement("!help", function (data) {
        const dd = $("#dropdown-special-commands");
        data.split("\n").forEach(function (line) {
            if (line.length) {
                const execute_cmd = function (ev) {
                    // "!cmd <arg> .. help comment" => "!cmd <arg>"
                    $("#stmt-input").val(line.replace(/\.\. .*$/g, ""));
                    $("#stmt-input").focus();
                };
                const new_item = $("<a href='#'>")
                    .addClass("dropdown-item")
                    .text(line)
                    .click(execute_cmd);
                dd.append(new_item);
            }
        });
    });

    // init
    clear_canvas();
    draw_grid_on_canvas();
    canvas.click(function (ev) {
        const rect = canvas.offset();
        const [logicalx, logicaly] = pixel2logical(
            Number.parseInt(ev.pageX - rect.left),
            Number.parseInt(ev.pageY - rect.top));
        const [pixelx, pixely] = logical2pixel(logicalx, logicaly);
        console.log("click x=" + logicalx + " y=" + logicaly);

        canvas_context.fillStyle = "rgb(128, 0, 0)";
        canvas_context.fillRect(pixelx, pixely, scale, scale);

        process_click(logicalx, logicaly);
    });

});
// vim:ts=2 sw=2 sts=2 et ci
