
{% for section in site.data.index.sections %}
## {{ section.title }}
{% for item in section.docs %}
 * [{{ item.title }}]({{ item.url }}) {% endfor %}
{% endfor %}
