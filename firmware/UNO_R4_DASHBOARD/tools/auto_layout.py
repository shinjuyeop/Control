"""Static AUTO cockpit assets."""

import math


WHITE = "#FFFFFF"
INK = "#131C25"
MUTED = "#687783"
RULE = "#CCD4DA"
DARK_RULE = "#45535F"
DARK_TEXT = "#B9C7D2"


class AutoLayout:
    def _auto_static_shell(self, draw):
        draw.rectangle((0, 0, 479, 319), fill=WHITE)
        draw.text((80, 69), "VEHICLE HEADING", font=self.fonts.medium(11), fill=MUTED, anchor="mm")
        draw.text((400, 69), "STEERING ANGLE", font=self.fonts.medium(11), fill=MUTED, anchor="mm")
        self._auto_compass_frame(draw, 80, 145, 52)
        self._auto_steering_frame(draw, 400, 145, 52)
        draw.text((240, 110), "SPEED", font=self.fonts.medium(11), fill=MUTED, anchor="mm")
        draw.text((240, 172), "KM/H", font=self.fonts.medium(11), fill=MUTED, anchor="mm")
        draw.text((240, 199), "TARGET SPEED", font=self.fonts.medium(11), fill=MUTED, anchor="mm")
        draw.rectangle((0,262,479,319),fill=INK)
        draw.text((10,273),"STEER CURRENT",font=self.fonts.medium(10),fill=DARK_TEXT,anchor="lm")
        for y, label in ((292, "OUTPUT CMD"), (309, "FEEDBACK")):
            draw.text((120,y),label,font=self.fonts.medium(10),fill=DARK_TEXT,anchor="lm")
        draw.line((309,301,470,301),fill=DARK_RULE)
        draw.text((305,275),"25A",font=self.fonts.medium(9),fill=DARK_TEXT,anchor="rm")

    def _auto_steering_frame(self, draw, cx, cy, radius):
        draw.ellipse((cx-radius, cy-radius, cx+radius, cy+radius), outline=RULE, width=2)
        draw.line((cx,cy-radius,cx,cy+radius),fill=RULE)
        draw.text((cx,cy-radius-10), "0°", font=self.fonts.display_bold(11), fill=INK, anchor="mm")
        draw.text((cx-radius-10,cy), "L", font=self.fonts.display_bold(11), fill=INK, anchor="mm")
        draw.text((cx+radius+10,cy), "R", font=self.fonts.display_bold(11), fill=INK, anchor="mm")
        for degree in (-120,-60,0,60,120):
            a=math.radians(-degree)
            draw.line((cx+math.sin(a)*(radius-7),cy-math.cos(a)*(radius-7),
                       cx+math.sin(a)*radius,cy-math.cos(a)*radius),fill=MUTED)
        draw.text((326,225),"ACTUAL",font=self.fonts.medium(11),fill=MUTED,anchor="lm")
        draw.text((326,249),"TARGET",font=self.fonts.medium(11),fill=MUTED,anchor="lm")

    def _auto_compass_frame(self, draw, cx, cy, radius):
        draw.ellipse((cx-radius, cy-radius, cx+radius, cy+radius), outline=RULE, width=2)
        for degree in range(0, 360, 30):
            angle = math.radians(degree)
            length = 8 if degree % 90 == 0 else 4
            draw.line((cx + math.sin(angle)*(radius-length), cy - math.cos(angle)*(radius-length),
                       cx + math.sin(angle)*radius, cy - math.cos(angle)*radius), fill=MUTED)
        for degree, label in ((0,"N"),(90,"E"),(180,"S"),(270,"W")):
            angle = math.radians(degree)
            draw.text((cx+math.sin(angle)*(radius+10), cy-math.cos(angle)*(radius+10)),
                      label, font=self.fonts.display_bold(11), fill=INK, anchor="mm")
