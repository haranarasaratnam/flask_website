(function(d) {
  // https://www.kirupa.com/html5/detect_whether_font_is_installed.htm
  var canvas = document.createElement("canvas");
  var context = canvas.getContext("2d");
  var text = "abcdefghijklmnopqrstuvwxyz0123456789";
  context.font = "72px monospace";
  var size = context.measureText(text).width;
  // songti
  var types = [' SC', ' CN', ' TC', ' TW', ''];
  for (var i = 0; i < types.length; i++) {
    context.font = "72px '" + 'Source Han Serif' + types[i] + "', monospace";
    if (context.measureText(text).width != size) return;
  }
  var config = {kitId: 'kwz5xar', scriptTimeout: 3000, async: true},
  h=d.documentElement,t=setTimeout(function(){h.className=h.className.replace(/\bwf-loading\b/g,"")+" wf-inactive";},config.scriptTimeout),tk=d.createElement("script"),f=false,s=d.getElementsByTagName("script")[0],a;h.className+=" wf-loading";tk.src='https://use.typekit.net/'+config.kitId+'.js';tk.async=true;tk.onload=tk.onreadystatechange=function(){a=this.readyState;if(f||a&&a!="complete"&&a!="loaded")return;f=true;clearTimeout(t);try{Typekit.load(config)}catch(e){}};s.parentNode.insertBefore(tk,s)
})(document);
