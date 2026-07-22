---
layout: archive
title: "Publications"
permalink: /publications/
author_profile: true
---

{% if author.googlescholar %}
  You can also find my articles on my <a href="{{author.googlescholar}}">Google Scholar profile</a>.
{% endif %}

{% include base_path %}

## Peer-Reviewed Publications

{% for post in site.publications reversed %}
  {% unless post.venue contains "preparation" %}
    {% include archive-single.html %}
  {% endunless %}
{% endfor %}

## In Preparation

{% for post in site.publications reversed %}
  {% if post.venue contains "preparation" %}
    {% include archive-single.html %}
  {% endif %}
{% endfor %}
