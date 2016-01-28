#include <json/json.h>
#include <stdio.h>
#include <string.h>
#include <curl/curl.h>


const char *wundergroundApiKey = "5418ef59f4a4c8fe";


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

int high;
int low;
char conditions[64];

void parse_object(json_object *obj)
{
	json_object_object_foreach(obj, key, val)
	{
		if (strcmp(key, "simpleforecast") == 0)
		{
			parse_object(val);
		}
		if (strcmp(key, "forecastday") == 0)
		{
			parse_array(val);
		}
		if (strcmp(key, "conditions") == 0)
		{
			const char *str = json_object_get_string(val);
			strcpy(conditions, str);
			printf("%s with a high of %d and a low of %d\n", str, high, low);
			exit(0);
		}
		if (strcmp(key, "date") == 0)
		{
			parse_object(val);
		}
		if (strcmp(key, "pretty") == 0)
		{
//			const char *str = json_object_get_string(val);
//			printf("\nDate: %s\n", str);
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



int main(int argc, char **argv)
{
    char result[8192];
    CURL *curl;
    CURLcode res;

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
	sprintf(url, "http://api.wunderground.com/api/%s/forecast/q/PA/Downingtown.json", wundergroundApiKey);
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

    // If there were no errors, parse the json response
    json_object * jobj = json_tokener_parse(result);
    enum json_type type;
    json_object_object_foreach(jobj, key, val)
    {
	type = json_object_get_type(val);
	if (type == json_type_object)
	{
	    if (strcmp(key, "forecast") == 0)
		parse_object(val);
	}
    }
}

