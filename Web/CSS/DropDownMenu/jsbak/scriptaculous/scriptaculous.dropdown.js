Event.observe(window, 'load', function() {
	$$(".dropdown li").each( function(li) {
		li.observe("mouseover", function(e) {
			li.addClassName("hover");
			if(li.down("ul") != undefined) {
				li.down("ul").setStyle( {
					visibility: "visible"
				});
			}
		});
		li.observe("mouseout", function(e) {
			li.removeClassName("hover");
			if(li.down("ul") != undefined) {
				li.down("ul").setStyle( {
					visibility: "hidden"
				});
			}
		});
	});
});