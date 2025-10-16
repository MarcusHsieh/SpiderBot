#include "rgb_led_control.h"
#include "WAIT1.h"
#include "CsIO1.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

//global vars
static LDD_TDeviceData *gpioc_device = NULL;
static bool rgb_led_initialized = false;
static RGBLed_RGB current_rgb1 = {0, 0, 0}; //start all off
static RGBLed_RGB current_rgb2 = {0, 0, 0}; //start all off

//color name strings
static const char* color_names[] = {
    "OFF",
    "RED", 
    "GREEN",
    "BLUE",
    "YELLOW",
    "MAGENTA", 
    "CYAN",
    "WHITE"
};

//send string to uart
static void rgb_puts(const char* str) {
    printf("%s", str);
}

//send character to uart
static void rgb_putchar(char ch) {
    printf("%c", ch);
}

//convert string to lowercase
static void str_tolower(char* str) {
    while (*str) {
        *str = tolower(*str);
        str++;
    }
}

RGBLed_Status RGBLed_Init(void) {
    //initialize gpioc component
    gpioc_device = GPIOC_Init(NULL);
    if (gpioc_device == NULL) {
        return RGB_LED_ERROR;
    }
    
    rgb_led_initialized = true;
    
    //set both leds to green initially
    RGBLed_Status status = RGBLed_SetColor(RGB_LED_BOTH, RGB_COLOR_GREEN);
    
    //print welcome message
    rgb_puts("\r\n=== RGB LED Control System Initialized ===\r\n");
    rgb_puts("Both LEDs set to GREEN\r\n");
    rgb_puts("Type 'help' for available commands\r\n\r\n");
    
    return status;
}

RGBLed_Status RGBLed_ColorToRGB(RGBLed_Color color, RGBLed_RGB *rgb) {
    if (rgb == NULL) {
        return RGB_LED_ERROR;
    }
    
    switch (color) {
        case RGB_COLOR_OFF:
            rgb->red = 0; rgb->green = 0; rgb->blue = 0;
            break;
        case RGB_COLOR_RED:
            rgb->red = 1; rgb->green = 0; rgb->blue = 0;
            break;
        case RGB_COLOR_GREEN:
            rgb->red = 0; rgb->green = 1; rgb->blue = 0;
            break;
        case RGB_COLOR_BLUE:
            rgb->red = 0; rgb->green = 0; rgb->blue = 1;
            break;
        case RGB_COLOR_YELLOW:
            rgb->red = 1; rgb->green = 1; rgb->blue = 0;
            break;
        case RGB_COLOR_MAGENTA:
            rgb->red = 1; rgb->green = 0; rgb->blue = 1;
            break;
        case RGB_COLOR_CYAN:
            rgb->red = 0; rgb->green = 1; rgb->blue = 1;
            break;
        case RGB_COLOR_WHITE:
            rgb->red = 1; rgb->green = 1; rgb->blue = 1;
            break;
        default:
            return RGB_LED_ERROR;
    }
    
    return RGB_LED_OK;
}

RGBLed_Status RGBLed_SetRGB(RGBLed_Selection led, RGBLed_RGB rgb) {
    if (!rgb_led_initialized || gpioc_device == NULL) {
        return RGB_LED_NOT_INITIALIZED;
    }
    
    //set rgb1 (led 1)
    if (led == RGB_LED_1 || led == RGB_LED_BOTH) {
        GPIOC_SetFieldValue(gpioc_device, RGB1_RED, rgb.red);
        GPIOC_SetFieldValue(gpioc_device, RGB1_GREEN, rgb.green);
        GPIOC_SetFieldValue(gpioc_device, RGB1_BLUE, rgb.blue);
        current_rgb1 = rgb;
    }
    
    //set rgb2 (led 2)
    if (led == RGB_LED_2 || led == RGB_LED_BOTH) {
        GPIOC_SetFieldValue(gpioc_device, RGB2_RED, rgb.red);
        GPIOC_SetFieldValue(gpioc_device, RGB2_GREEN, rgb.green);
        GPIOC_SetFieldValue(gpioc_device, RGB2_BLUE, rgb.blue);
        current_rgb2 = rgb;
    }
    
    return RGB_LED_OK;
}

RGBLed_Status RGBLed_SetColor(RGBLed_Selection led, RGBLed_Color color) {
    RGBLed_RGB rgb;
    RGBLed_Status status = RGBLed_ColorToRGB(color, &rgb);
    
    if (status != RGB_LED_OK) {
        return status;
    }
    
    return RGBLed_SetRGB(led, rgb);
}

RGBLed_Status RGBLed_TurnOff(RGBLed_Selection led) {
    return RGBLed_SetColor(led, RGB_COLOR_OFF);
}

RGBLed_Status RGBLed_GetRGB(RGBLed_Selection led, RGBLed_RGB *rgb) {
    if (!rgb_led_initialized || rgb == NULL) {
        return RGB_LED_ERROR;
    }
    
    if (led == RGB_LED_1) {
        *rgb = current_rgb1;
    } else if (led == RGB_LED_2) {
        *rgb = current_rgb2;
    } else {
        return RGB_LED_ERROR;
    }
    
    return RGB_LED_OK;
}

const char* RGBLed_GetColorName(RGBLed_Color color) {
    if (color >= RGB_COLOR_OFF && color <= RGB_COLOR_WHITE) {
        return color_names[color];
    }
    return "UNKNOWN";
}

void RGBLed_PrintHelp(void) {
    rgb_puts("\r\n=== RGB LED Control Commands ===\r\n");
    rgb_puts("Available colors: off, red, green, blue, yellow, magenta, cyan, white\r\n");
    rgb_puts("Note: Use dot prefix (e.g., .cyan) to avoid conflicts with servo commands\r\n\r\n");
    rgb_puts("Commands:\r\n");
    rgb_puts("  <color>           - Set both LEDs to color\r\n");
    rgb_puts("  .<color>          - Set both LEDs to color (with dot prefix)\r\n");
    rgb_puts("  1 <color>         - Set LED 1 to color\r\n");
    rgb_puts("  2 <color>         - Set LED 2 to color\r\n");
    rgb_puts("  status            - Show current LED colors\r\n");
    rgb_puts("  test              - Run color test sequence\r\n");
    rgb_puts("  help              - Show this help\r\n");
    rgb_puts("\r\nExamples:\r\n");
    rgb_puts("  red               - Both LEDs red\r\n");
    rgb_puts("  .cyan             - Both LEDs cyan (avoids servo conflict)\r\n");
    rgb_puts("  1 blue            - LED 1 blue\r\n");
    rgb_puts("  2 green           - LED 2 green\r\n\r\n");
}

//find color from string
static RGBLed_Color string_to_color(const char* color_str) {
    char temp[16];
    strncpy(temp, color_str, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';
    str_tolower(temp);
    
    //handle both w and w/o period prefix
    const char* color_name = temp;
    if (temp[0] == '.') {
        color_name = &temp[1];
    }
    
    if (strcmp(color_name, "off") == 0) return RGB_COLOR_OFF;
    if (strcmp(color_name, "red") == 0) return RGB_COLOR_RED;
    if (strcmp(color_name, "green") == 0) return RGB_COLOR_GREEN;
    if (strcmp(color_name, "blue") == 0) return RGB_COLOR_BLUE;
    if (strcmp(color_name, "yellow") == 0) return RGB_COLOR_YELLOW;
    if (strcmp(color_name, "magenta") == 0) return RGB_COLOR_MAGENTA;
    if (strcmp(color_name, "cyan") == 0) return RGB_COLOR_CYAN;
    if (strcmp(color_name, "white") == 0) return RGB_COLOR_WHITE;
    
    return (RGBLed_Color)-1; //invalid color
}

//get curr color name
static const char* get_current_color_name(RGBLed_Selection led) {
    RGBLed_RGB rgb;
    if (RGBLed_GetRGB(led, &rgb) != RGB_LED_OK) {
        return "ERROR";
    }
    
    //check all colors
    for (int color = RGB_COLOR_OFF; color <= RGB_COLOR_WHITE; color++) {
        RGBLed_RGB check_rgb;
        if (RGBLed_ColorToRGB((RGBLed_Color)color, &check_rgb) == RGB_LED_OK) {
            if (check_rgb.red == rgb.red && 
                check_rgb.green == rgb.green && 
                check_rgb.blue == rgb.blue) {
                return RGBLed_GetColorName((RGBLed_Color)color);
            }
        }
    }
    
    return "CUSTOM";
}

RGBLed_Status RGBLed_ProcessCommand(const char* command) {
    if (!rgb_led_initialized) {
        rgb_puts("Error: RGB LED system not initialized\r\n");
        return RGB_LED_NOT_INITIALIZED;
    }
    
    char cmd_copy[64];
    strncpy(cmd_copy, command, sizeof(cmd_copy) - 1);
    cmd_copy[sizeof(cmd_copy) - 1] = '\0';
    
    char *p = cmd_copy;
    while (*p && *p != '\r' && *p != '\n') p++;
    *p = '\0';
    p = cmd_copy;
    while (*p && *p == ' ') p++;
    
    if (strlen(p) == 0) {
        return RGB_LED_OK;
    }
    
    char lower_cmd[64];
    strncpy(lower_cmd, p, sizeof(lower_cmd) - 1);
    lower_cmd[sizeof(lower_cmd) - 1] = '\0';
    str_tolower(lower_cmd);

    char *token1 = strtok(lower_cmd, " ");
    char *token2 = strtok(NULL, " ");
    
    if (token1 == NULL) {
        return RGB_LED_OK;
    }
    
    //help
    if (strcmp(token1, "help") == 0) {
        RGBLed_PrintHelp();
        return RGB_LED_OK;
    }
    
    //status
    if (strcmp(token1, "status") == 0) {
        rgb_puts("Current LED Status:\r\n");
        printf("  LED 1: %s\r\n", get_current_color_name(RGB_LED_1));
        printf("  LED 2: %s\r\n", get_current_color_name(RGB_LED_2));
        rgb_puts("\r\n");
        return RGB_LED_OK;
    }
    
    //test
    if (strcmp(token1, "test") == 0) {
        rgb_puts("Running color test sequence...\r\n");
        return RGBLed_TestSequence(1000);
    }
    
    if (strcmp(token1, "1") == 0 && token2 != NULL) {
        RGBLed_Color color = string_to_color(token2);
        if (color == (RGBLed_Color)-1) {
            printf("Error: Unknown color '%s'\r\n", token2);
            return RGB_LED_ERROR;
        }
        
        RGBLed_Status status = RGBLed_SetColor(RGB_LED_1, color);
        if (status == RGB_LED_OK) {
            printf("LED 1 set to %s\r\n", RGBLed_GetColorName(color));
        }
        return status;
    }
    
    if (strcmp(token1, "2") == 0 && token2 != NULL) {
        RGBLed_Color color = string_to_color(token2);
        if (color == (RGBLed_Color)-1) {
            printf("Error: Unknown color '%s'\r\n", token2);
            return RGB_LED_ERROR;
        }
        
        RGBLed_Status status = RGBLed_SetColor(RGB_LED_2, color);
        if (status == RGB_LED_OK) {
            printf("LED 2 set to %s\r\n", RGBLed_GetColorName(color));
        }
        return status;
    }
    
    RGBLed_Color color = string_to_color(token1);
    if (color != (RGBLed_Color)-1) {
        RGBLed_Status status = RGBLed_SetColor(RGB_LED_BOTH, color);
        if (status == RGB_LED_OK) {
            printf("Both LEDs set to %s\r\n", RGBLed_GetColorName(color));
        }
        return status;
    }
    
    printf("Error: Unknown command '%s'. Type 'help' for available commands.\r\n", token1);
    return RGB_LED_ERROR;
}

RGBLed_Status RGBLed_TestSequence(uint16_t delay_ms) {
    if (!rgb_led_initialized) {
        return RGB_LED_NOT_INITIALIZED;
    }
    
    RGBLed_Color colors[] = {
        RGB_COLOR_RED,
        RGB_COLOR_GREEN, 
        RGB_COLOR_BLUE,
        RGB_COLOR_YELLOW,
        RGB_COLOR_MAGENTA,
        RGB_COLOR_CYAN,
        RGB_COLOR_WHITE,
        RGB_COLOR_OFF
    };
    
    int num_colors = sizeof(colors) / sizeof(colors[0]);
    
    for (int i = 0; i < num_colors; i++) {
        printf("Test: %s\r\n", RGBLed_GetColorName(colors[i]));
        RGBLed_SetColor(RGB_LED_BOTH, colors[i]);
        WAIT1_Waitms(delay_ms);
    }
    
    rgb_puts("Test sequence complete\r\n");
    return RGB_LED_OK;
} 