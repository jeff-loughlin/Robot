#include <json/json.h>
#include <stdio.h>
#include <string.h>
#include <curl/curl.h>


int high;
int low;
char conditions[256];
bool flag_show_all = false;
int nPeriods = 99;
int period = 0;

//
// NOTE:  parse_array() and parse_object() are a quick and dirty way to parse the json we get back from wunderground.com
//        It gets me what I need, but there are prettier and more robust ways to do this parsing...
//
void parse_object(json_object *obj);
void parse_array(json_object *obj)
{
    array_list *arr = json_object_get_array(obj);
    int n = 0;
    for (n = 0; n < array_list_length(arr); n++)
    {
	json_object *o = (json_object *)array_list_get_idx(arr, n);
	parse_object(o);
    }
}

void parse_object(json_object *obj)
{
    json_object_object_foreach(obj, key, val)
    {
	if (flag_show_all)
	{
	    if (strcmp(key, "fcttext") == 0 && flag_show_all)
	    {
		const char *str = json_object_get_string(val);
		printf("    %s\n", str);
		period++;
		if (period == nPeriods)
		    exit(0);
	    }
	    if (strcmp(key, "title") == 0 && flag_show_all)
	    {
		const char *str = json_object_get_string(val);
		printf("\n%s\n", str);
	    }
	}
	else
	{
	    if (strcmp(key, "conditions") == 0)
	    {
		const char *str = json_object_get_string(val);
		strcpy(conditions, str);
		printf("%s with a high of %d and a low of %d\n", str, high, low);
		exit(0);
	    }
	    if (strcmp(key, "date") == 0 && !flag_show_all)
	    {
		parse_object(val);
	    }
	    if (strcmp(key, "high") == 0)
	    {
		json_object_object_foreach(val, k, v)
		{
		    if (strcmp(k, "fahrenheit") == 0)
		    {
			const char *str = json_object_get_string(v);
			high = atoi(str);
		    }
		}
	    }
	    if (strcmp(key, "low") == 0)
	    {
		json_object_object_foreach(val, k, v)
		{
		    if (strcmp(k, "fahrenheit") == 0)
		    {
			const char *str = json_object_get_string(v);
			low = atoi(str);
		    }
		}
	    }
	}
	if (strcmp(key, "txt_forecast") == 0 && flag_show_all)
	{
	    parse_object(val);
	}
	if (strcmp(key, "simpleforecast") == 0)
	{
	    parse_object(val);
	}
	if (strcmp(key, "forecastday") == 0)
	{
	    parse_array(val);
	}
    }
}



struct MemoryStruct
{
    char *memory;
    size_t size;
};

static size_t WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    size_t realsize = size * nmemb;
    struct MemoryStruct *mem = (struct MemoryStruct *)userp;

    mem->memory = (char *)realloc(mem->memory, mem->size + realsize + 1);
    if(mem->memory == NULL)
    {
	/* out of memory! */
	printf("not enough memory (realloc returned NULL)\n");
	return 0;
    }

    memcpy(&(mem->memory[mem->size]), contents, realsize);
    mem->size += realsize;
    mem->memory[mem->size] = 0;

    return realsize;
}


void parseConfigFile(char *apiKey, char *city, char *state)
{
    // Get the city, state, and API key for wunderground.com from the rc file
    // NOTE:  This is a quick-and-dirty routine and doesn't handle a malformed .weatherrc file very well.
    //        Lots of room for improvement here
    //
    const char *home = getenv("HOME");
    char filename[256];
    sprintf(filename,"%s/.weatherrc", home);
    FILE *f = fopen(filename, "r");
    if (f == 0)
    {
	printf("Can't open ~/.weatherrc");
	exit(-1);
    }

    while (!feof(f))
    {
	char line[256];
	fgets(line, 256, f);
	char *key = strtok(line, "=");
	char *value = strtok(0, "=");

	if (value != 0)
	{
	    if (value[strlen(value) - 1] == '\n')
		value[strlen(value) - 1] = '\0';

	    if (strcmp(key, "key") == 0)
		strcpy(apiKey, value);
	    else if (strcmp(key, "city") == 0)
		strcpy(city, value);
	    else if (strcmp(key, "state") == 0)
		strcpy(state, value);
	}
    }
}

int main(int argc, char **argv)
{
    char result[8192];
    CURL *curl;
    CURLcode res;

    if (argc == 2)
    {
	flag_show_all = true;
	if (strcmp(argv[1], "-a") == 0)
	    nPeriods = 99;
	else
	{
	    char numStr[32];
	    strcpy(numStr,&(argv[1][1]));
	    nPeriods = atoi(numStr);
	}
    }

    char apiKey[64];
    char city[64];
    char state[32];
    parseConfigFile(apiKey, city, state);

    struct MemoryStruct chunk;

    chunk.memory = (char *)malloc(1);  // will be grown as needed by the realloc in WriteMemoryCallback()
    chunk.size = 0;    // no data at this point

    curl_global_init(CURL_GLOBAL_ALL);
    curl = curl_easy_init();
    if (curl)
    {
	/* First set the URL that is about to receive our POST. This URL can
	   just as well be a https:// URL if that is what should receive the
	   data. */
	char url[256];
	sprintf(url, "http://api.wunderground.com/api/%s/forecast/q/%s/%s.json", apiKey, state, city);
	curl_easy_setopt(curl, CURLOPT_URL, url);
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&chunk);


	// Perform the request, res will get the return code
	res = curl_easy_perform(curl);
	if (res != CURLE_OK)
	{
	    fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
	    return -1;
	}
	strcpy(result, chunk.memory);

	curl_easy_cleanup(curl);
    }
    curl_global_cleanup();

    // If there were no errors, parse the json response - looking for a top-level object with key "forecast"
    json_object *jobj = json_tokener_parse(result);
    enum json_type type;
    json_object_object_foreach(jobj, key, val)
    {
	type = json_object_get_type(val);
	if (type == json_type_object)
	{
	    // Once we find the "forcast" object, parse it
	    if (strcmp(key, "forecast") == 0)
		parse_object(val);
	}
    }
}

