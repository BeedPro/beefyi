import { DateTime } from "luxon";

function formatReadingTime(totalMinutes) {
	return `${totalMinutes} min read`;
}

function stripCodeBlocksAndHtml(html) {
	return (html || "")
		.replace(/<pre\b[^>]*>[\s\S]*?<\/pre>/gi, " ")
		.replace(/<code\b[^>]*>[\s\S]*?<\/code>/gi, " ")
		.replace(/<script\b[^>]*>[\s\S]*?<\/script>/gi, " ")
		.replace(/<style\b[^>]*>[\s\S]*?<\/style>/gi, " ")
		.replace(/<!--([\s\S]*?)-->/g, " ")
		.replace(/<[^>]+>/g, " ")
		.replace(/&nbsp;/gi, " ")
		.replace(/&#160;/gi, " ")
		.replace(/&[a-z0-9#]+;/gi, " ")
		.replace(/\s+/g, " ")
		.trim();
}

export default function(eleventyConfig) {
	eleventyConfig.addFilter("readableDate", (dateObj, format, zone) => {
		// Formatting tokens for Luxon: https://moment.github.io/luxon/#/formatting?id=table-of-tokens
		return DateTime.fromJSDate(dateObj, { zone: zone || "utc" }).toFormat(format || "dd LLLL yyyy");
	});

	eleventyConfig.addFilter("htmlDateString", (dateObj) => {
		// dateObj input: https://html.spec.whatwg.org/multipage/common-microsyntaxes.html#valid-date-string
		return DateTime.fromJSDate(dateObj, { zone: "utc" }).toFormat('yyyy-LL-dd');
	});

	// Get the first `n` elements of a collection.
	eleventyConfig.addFilter("head", (array, n) => {
		if(!Array.isArray(array) || array.length === 0) {
			return [];
		}
		if( n < 0 ) {
			return array.slice(n);
		}

		return array.slice(0, n);
	});

	// Return the smallest number argument
	eleventyConfig.addFilter("min", (...numbers) => {
		return Math.min.apply(null, numbers);
	});

	// Return the keys used in an object
	eleventyConfig.addFilter("getKeys", target => {
		return Object.keys(target);
	});

	eleventyConfig.addFilter("filterTagList", function filterTagList(tags) {
		return (tags || []).filter(tag => ["all", "posts"].indexOf(tag) === -1);
	});

	eleventyConfig.addFilter("sortAlphabetically", strings =>
		(strings || []).sort((b, a) => b.localeCompare(a))
	);

	eleventyConfig.addFilter("readingTime", html => {
		const text = stripCodeBlocksAndHtml(html);

		if(!text) {
			return formatReadingTime(1);
		}

		const words = text.split(/\s+/).filter(Boolean).length;
		const totalMinutes = Math.max(1, Math.ceil(words / 200));

		return formatReadingTime(totalMinutes);
	});
};
